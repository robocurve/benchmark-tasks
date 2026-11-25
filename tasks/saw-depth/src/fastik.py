"""Fast YAM arm kinematics (NumPy): the same URDF chain and TCP as ``yam_arena.ik`` with a precomputed chain and a
geometric Jacobian, so the closed-loop demo can re-solve IK every control step cheaply. ``check()`` compares it with
``yam_arena.ik.tcp_pose``."""

from __future__ import annotations

from functools import lru_cache

import numpy as np


@lru_cache(maxsize=1)
def _chain():
    from yam_arena import YAM_URDF
    from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
    from yam_arena.urdf import parse_joints, rpy_matrix

    j = parse_joints(YAM_URDF)
    by_child = {v.child: v for v in j.values()}
    names, link = [], "gripper"
    while link in by_child:
        jt = by_child[link]
        names.append(jt.name)
        link = jt.parent
    names = names[::-1]
    chain = []
    for n in names:
        jt = j[n]
        t = np.eye(4)
        t[:3, :3] = rpy_matrix(jt.rpy)
        t[:3, 3] = jt.xyz
        chain.append((n, t, jt.axis / np.linalg.norm(jt.axis), jt.type))
    lower = np.array([j[n].lower for n in ARM_JOINTS])
    upper = np.array([j[n].upper for n in ARM_JOINTS])
    return chain, np.array(TCP_OFFSET, float), lower, upper, list(ARM_JOINTS)


def _axis_angle(a, ang):
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * k + (1 - np.cos(ang)) * (k @ k)


def fk(q, base=(0.0, 0.0, 0.0)):
    """TCP (pos, rot) and per-joint (axis_world, origin_world) for the Jacobian."""
    chain, tcp, _, _, arm = _chain()
    qd = dict(zip(arm, q))
    t = np.eye(4)
    t[:3, 3] = base
    axes = []
    for n, to, ax, typ in chain:
        t = t @ to
        if typ in ("revolute", "continuous"):
            axes.append((t[:3, :3] @ ax, t[:3, 3].copy()))
            r = np.eye(4)
            r[:3, :3] = _axis_angle(ax, qd.get(n, 0.0))
            t = t @ r
    p = t[:3, :3] @ tcp + t[:3, 3]
    return p, t[:3, :3], axes


def tcp_pose(q, base=(0.0, 0.0, 0.0)):
    p, r, _ = fk(q, base)
    return p, r


def _rot_err(rt, r):
    m = rt @ r.T
    c = np.clip((np.trace(m) - 1) / 2, -1, 1)
    ang = np.arccos(c)
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    s = np.linalg.norm(v)
    if s < 1e-9:  # 180 deg
        w, u = np.linalg.eigh((m + np.eye(3)) / 2)
        return u[:, -1] * np.pi
    return v / s * ang


def solve(
    pos, rot, q0, base=(0.0, 0.0, 0.0), iters=60, damping=0.02, rot_weight=0.3, tol=2e-5
):
    """DLS IK to (pos, rot). Returns (q, pos_err, rot_err)."""
    _, _, lo, hi, _ = _chain()
    q = np.array(q0, float)
    pos = np.asarray(pos, float)
    for _ in range(iters):
        p, r, axes = fk(q, base)
        e = np.r_[pos - p, rot_weight * _rot_err(rot, r)]
        if np.linalg.norm(e) < tol:
            break
        jac = np.zeros((6, 6))
        for i, (a, o) in enumerate(axes):
            jac[:3, i] = np.cross(a, p - o)
            jac[3:, i] = rot_weight * a
        dq = jac.T @ np.linalg.solve(jac @ jac.T + damping**2 * np.eye(6), e)
        q = np.clip(q + dq, lo, hi)
    p, r, _ = fk(q, base)
    return q, float(np.linalg.norm(pos - p)), float(np.linalg.norm(_rot_err(rot, r)))


def check(n=20, seed=0):
    from yam_arena import ik

    rng = np.random.default_rng(seed)
    _, _, lo, hi, _ = _chain()
    worst = 0.0
    for _ in range(n):
        q = rng.uniform(lo, hi)
        p0, r0 = ik.tcp_pose(q, (0, 0.305, 0.75))
        p1, r1 = tcp_pose(q, (0, 0.305, 0.75))
        worst = max(worst, float(np.abs(p0 - p1).max()), float(np.abs(r0 - r1).max()))
    return worst
