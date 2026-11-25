"""One YAM arm's URDF kinematics (NumPy): forward kinematics, the geometric Jacobian at the tool point, and the force the
arm applies there from its joint torques. Matches yam_arena.ik.tcp_pose. Frames: the arm's station frame."""

from __future__ import annotations

import math

import numpy as np
from yam_arena import YAM_URDF
from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
from yam_arena.station import BASES
from yam_arena.urdf import parse_joints, rpy_matrix

_J = parse_joints(YAM_URDF)
_CHAIN = [_J[n] for n in ARM_JOINTS]
_ORIG = []
for _j in _CHAIN:
    _T = np.eye(4)
    _T[:3, :3] = rpy_matrix(_j.rpy)
    _T[:3, 3] = _j.xyz
    _ORIG.append(_T)
_AX = [j.axis / np.linalg.norm(j.axis) for j in _CHAIN]
_TCP = np.array(TCP_OFFSET)


def _rot(axis, a):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


def fk(q, side: str):
    """(joint axes, joint points, tool point, tool rotation) in the station frame."""
    T = np.eye(4)
    T[:3, 3] = BASES[side]
    axes, pts = [], []
    for i in range(6):
        T = T @ _ORIG[i]
        axes.append(T[:3, :3] @ _AX[i])
        pts.append(T[:3, 3].copy())
        Rj = np.eye(4)
        Rj[:3, :3] = _rot(_AX[i], q[i])
        T = T @ Rj
    return axes, pts, T[:3, :3] @ _TCP + T[:3, 3], T[:3, :3]


def jacobian(q, side: str) -> np.ndarray:
    axes, pts, p, _ = fk(q, side)
    J = np.zeros((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(axes[i], p - pts[i])
        J[3:, i] = axes[i]
    return J


def wrench(q, tau, side: str) -> np.ndarray:
    """The wrench (force, moment about the tool point) the arm applies at its tool point for joint torques ``tau``."""
    J = jacobian(q, side)
    return np.linalg.lstsq(J.T, np.asarray(tau, float), rcond=1e-6)[0]


def support_force(q, tau, side: str) -> float:
    """Upward force the arm applies at its tool point (N)."""
    return float(wrench(q, tau, side)[2])


def preload(q, force_up: float, side: str, stiffness) -> np.ndarray:
    """Joint-target offset that makes a PD arm of ``stiffness`` push up with ``force_up`` (N) at its tool point."""
    J = jacobian(q, side)
    tau = J.T @ np.array([0.0, 0.0, force_up, 0.0, 0.0, 0.0])
    return tau / np.asarray(stiffness, float)
