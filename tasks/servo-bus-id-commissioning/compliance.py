"""Cartesian compliance of a YAM arm (soft joint PD controllers) and the resulting static cap travel of a push button.

The arms are driven by joint PD controllers with finite gains (joints 1-3: 80 N m/rad, joints 4-6: 10 N m/rad, from
yam_arena.constants, gravity compensated). A force F at the fingertip therefore moves the tip by C F with
C = J K^-1 J^T (6x6 at the tip point). Pressing a cap of spring rate k with the tip COMMANDED ``depth`` below the cap's
rest top reaches the static equilibrium

    travel = depth / (1 + C k)         while that is below the end stop,
    travel = stop                      otherwise (the stop, not the arm, then sets the travel).

Numpy only; the Jacobian is taken numerically from yam_arena.ik.tcp_pose.
"""

from __future__ import annotations

import numpy as np

# Joint stiffness (N m/rad) of joints 1-6 (yam_arena.constants SHOULDER / WRIST)
K_JOINT = np.diag([80.0, 80.0, 80.0, 10.0, 10.0, 10.0])
# Joint friction (N m) of joints 1-6: the arm stops short of a target by up to friction/stiffness per joint
FRICTION = np.array([0.3, 0.3, 0.3, 0.06, 0.06, 0.06])


def tip_jacobian(q, base, tip_offset_world=(0.0, 0.0, -0.013)) -> np.ndarray:
    """6x6 Jacobian (v; w) of the point ``tip_offset_world`` (world vector from the TCP) w.r.t. joints 1-6."""
    from yam_arena import ik

    q = np.asarray(q, float)
    p0, r0 = ik.tcp_pose(q, base)
    jac = np.zeros((6, 6))
    eps = 1e-6
    off = np.asarray(tip_offset_world, float)
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        p1, r1 = ik.tcp_pose(dq, base)
        d = r1 @ r0.T
        w = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]]) / 2 / eps
        jac[:3, i] = (p1 - p0) / eps + np.cross(w, off)
        jac[3:, i] = w
    return jac


def vertical_compliance(q, base, tip_offset_world=(0.0, 0.0, -0.013)) -> float:
    """Vertical tip compliance C_zz in m/N for joint angles q."""
    jac = tip_jacobian(q, base, tip_offset_world)
    return float((jac @ np.linalg.inv(K_JOINT) @ jac.T)[2, 2])


def friction_allowance(q, base, tip_offset_world=(0.0, 0.0, -0.013)) -> float:
    """Upper bound (m) of how far joint friction can stop the tip short of its command (vertical)."""
    jac = tip_jacobian(q, base, tip_offset_world)
    return float(np.abs(jac[2, :] * (FRICTION / np.diag(K_JOINT))).sum())


def equilibrium_travel(compliance_m_per_n: float, cmd_depth: float, rate: float, stop: float, short_by: float = 0.0) -> float:
    """Static cap travel (m) when the tip is commanded ``cmd_depth`` (m) below the rest cap top.

    ``short_by`` models joint friction stopping the arm short of its command (m, reduces the effective depth).
    """
    depth = max(cmd_depth - short_by, 0.0)
    return float(min(stop, depth / (1.0 + compliance_m_per_n * rate)))
